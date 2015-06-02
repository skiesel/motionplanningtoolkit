// This is taken from https://github.com/eaburns/search
// It's a combination of some of the various functionality in the utils subdirectory associated
// with creating a datafile easily -- thanks eaburns
// © 2013 the Search Authors under the MIT license. See AUTHORS for the list of authors.

#include <cstdarg>
#include <cstdio>
#include <ctime>
#include <cerrno>
#include <cstring>
#include <cassert>
#include <sys/utsname.h>
#include <sys/stat.h>

enum { Bufsz = 256 };

const char *start4 = "#start data file format 4\n";
const char *end4 = "#end data file format 4\n";

static void dfpair_sz(FILE*, unsigned int, const char*, const char*, va_list);
static void machineid(FILE*);
static void tryprocstatus(FILE*);

void warn(const char *fmt, ...) {
	char mbuf[Bufsz];

	va_list args;
	va_start(args, fmt);
	vsnprintf(mbuf, Bufsz, fmt, args);
	va_end(args);

	fprintf(stderr, "%s\n", mbuf);
}

void warnx(int err, const char *fmt, ...) {
	char mbuf[Bufsz];

	va_list args;
	va_start(args, fmt);
	vsnprintf(mbuf, Bufsz, fmt, args);
	va_end(args);

	warn("%s: %s", mbuf, strerror(err));
}

void fatal(const char *fmt, ...) {
	char mbuf[Bufsz];

	va_list args;
	va_start(args, fmt);
	vsnprintf(mbuf, Bufsz, fmt, args);
	va_end(args);

	fprintf(stderr, "%s\n", mbuf);
	exit(1);
}

void fatalx(int err, const char *fmt, ...) {
	char mbuf[Bufsz];

	va_list args;
	va_start(args, fmt);
	vsnprintf(mbuf, Bufsz, fmt, args);
	va_end(args);

	fatal("%s: %s", mbuf, strerror(err));
}

bool fileexists(const std::string &path) {
	struct stat sb;
	if (stat(path.c_str(), &sb) == -1) {
		if (errno == ENOENT)
			return false;
		fatalx(errno, "failed to stat %s", path.c_str());
	}
	return true;
}

bool hasprefix(const char *str, const char *prefix) {
	unsigned int plen = strlen(prefix);
	if (strlen(str) < plen)
		return false;
	for (unsigned int i = 0; i < plen; i++)
		if (str[i] != prefix[i])
			return false;
	return true;
}

double walltime() {
	struct timeval tv;

	if (gettimeofday(&tv, NULL) == -1)
		fatalx(errno, "gettimeofday failed");

	return (double) tv.tv_sec + (double) tv.tv_usec / 1000000.0;
}

void dfpair(FILE *f, const char *key, const char *fmt, ...) {
	char buf[Bufsz];
	int n = snprintf(buf, Bufsz, "#pair  \"%s\"\t\"", key);
	if (n > Bufsz)
		fatal("dfrowhdr: buffer is too small\n");

	va_list ap;
	va_start(ap, fmt);
	unsigned int m = vsnprintf(buf+n, Bufsz-n, fmt, ap);
	va_end(ap);

	if (m > (unsigned int) Bufsz-n) {	// Err, overflow
		va_start(ap, fmt);
		dfpair_sz(f, m + n + 1, key, fmt, ap);
		va_end(ap);
		return;
	}

	fprintf(f, "%s\"\n", buf);
	fflush(f);
}

static void dfpair_sz(FILE *f, unsigned int sz, const char *key, const char *fmt, va_list ap) {
	char *buf = new char[sz];

	unsigned int n = snprintf(buf, sz, "#pair  \"%s\"\t\"", key);
	assert (n <= sz);
	vsnprintf(buf+n, sz-n, fmt, ap);

	fprintf(f, "%s\"\n", buf);
	delete[] buf;
}

void dfrowhdr(FILE *f, const char *name, unsigned int ncols, ...) {
	char buf[Bufsz];
	int n = snprintf(buf, Bufsz, "#altcols  \"%s\"", name);
	if (n > Bufsz)
		fatal("dfrowhdr: buffer is too small\n");

	va_list ap;
	va_start(ap, ncols);
	for (unsigned int i = 0; i < ncols; i++) {
		char *col = va_arg(ap, char*);
		unsigned int m = snprintf(buf+n, Bufsz-n, "\t\"%s %s\"", name, col);
		if (m > (unsigned int) Bufsz - n)
			fatal("dfrowhdr: buffer is too small\n");
		n += m;
	}
	va_end(ap);

	fprintf(f, "%s\n", buf);
}

void dfrow(FILE *f, const char *name, const char *colfmt, ...) {
	char buf[Bufsz];
	int n = snprintf(buf, Bufsz, "#altrow  \"%s\"", name);
	if (n > Bufsz)
		fatal("dfrowhdr: buffer is too small\n");

	va_list ap;
	va_start(ap, colfmt);
	for (unsigned int i = 0; i < strlen(colfmt); i++) {
		double g = 0;
		long d = 0;
		unsigned long u = 0;
		unsigned int m = 0;
		switch (colfmt[i]) {
		case 'g':
			g = va_arg(ap, double);
			m = snprintf(buf+n, Bufsz-n, "\t%g", g);
			break;
		case 'f':
			g = va_arg(ap, double);
			m = snprintf(buf+n, Bufsz-n, "\t%lf", g);
			break;
		case 'd':
			d = va_arg(ap, long);
			m = snprintf(buf+n, Bufsz-n, "\t%ld", d);
			break;
		case 'u':
			u = va_arg(ap, unsigned long);
			m = snprintf(buf+n, Bufsz-n, "\t%lu", u);
			break;
		}
		if (m > (unsigned int) Bufsz-n)
			fatal("dfrow: buffer is too small\n");
		n += m;
	}
	va_end(ap);

	fprintf(f, "%s\n", buf);
	fflush(f);
}

void dfheader(FILE *f) {
	fputs(start4, f);

	time_t tm;
	time(&tm);
	char *tstr = ctime(&tm);
	tstr[strlen(tstr)-1] = '\0';
	dfpair(f, "wall start date", "%s", tstr);

	dfpair(f, "wall start time", "%g", walltime());
	machineid(f);
}

void dffooter(FILE *f) {
	dfpair(f, "wall finish time", "%g", walltime());

	time_t tm;
	time(&tm);
	char *tstr = ctime(&tm);
	tstr[strlen(tstr)-1] = '\0';
	dfpair(f, "wall finish date", "%s", tstr);
	tryprocstatus(f);
	fputs(end4, f);
}

void dfprocstatus(FILE *f) {
	tryprocstatus(f);
}

static void machineid(FILE *f) {
	char hname[Bufsz];
	memset(hname, '\0', Bufsz);
	if (gethostname(hname, Bufsz) == -1) {
		warnx(errno, "gethostname failed, unable to print machine id\n");
		return;
	}

	struct utsname u;
	if (uname(&u) == -1) {
		warnx(errno, "uname failed\n");
		dfpair(f, "machine id", "%s", hname);
	}

	dfpair(f, "machine id", "%s-%s-%s-%s", hname, u.sysname, u.release, u.machine);
}

static void tryprocstatus(FILE *out)
{
	static const char *field = "VmPeak:";
	static const char *key = "max virtual kilobytes";
	char buf[Bufsz];

	int n = snprintf(buf, Bufsz, "/proc/%d/status", getpid());
	if (n <= 0 || n > Bufsz)
		return;

	if (!fileexists(buf))
		return;

	FILE *in = fopen(buf, "r");
	if (!in)
		fatalx(errno, "Failed to open %s for reading", buf);

	for (;;) {
		if (!fgets(buf, Bufsz, in))
			break;
		if (!strstr(buf, field))
			continue;
		size_t skip = strspn(buf + strlen(field), " \t");
		char *strt = buf + strlen(field) + skip;
		char *end = strchr(strt, ' ');
		*end = '\0';
		dfpair(out, key, "%s", strt);
		break;
	}

	fclose(in);
}

char *strdup(const std::string &s) {
	char *d = new char[s.size() + 1];
	strcpy(d, s.c_str());
	return d;
}

// void dfread(FILE *in, Dfhandler seeline, void *priv, FILE *echo) {
// 	unsigned int lineno = 1;
// 	std::vector<std::string> toks;
// 	boost::optional<std::string> line = readline(in, echo);

// 	while (line) {
// 		char *linebuf = strdup(*line);
// 		toks.clear();

// 		if (hasprefix(linebuf, "#pair"))
// 			toks.push_back("#pair");
// 		else if (hasprefix(linebuf, "#altcols"))
// 			toks.push_back("#altcols");
// 		else if (hasprefix(linebuf, "#altrow"))
// 			toks.push_back("#altrow");

// 		if (toks.size() > 0) {
// 			const char *end = linebuf + strlen(linebuf);
// 			char *left = linebuf + toks[0].size();
// 			while (left < end) {
// 				char *vl = gettoken(left, lineno);
// 				if (!vl)
// 					break;
// 				toks.push_back(vl);
// 				left = vl + strlen(vl) + 1;
// 			}
// 			seeline(toks, priv);
// 		}

// 		delete[] linebuf;
// 		lineno++;
// 		line = readline(in, echo);
// 	}
// }